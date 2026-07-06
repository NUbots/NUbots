/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <algorithm>

#include "Director.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;

    bool Director::merge_pushes(Evaluation& parent, const Evaluation& child) {
            // First pushes to arrive are taken as is
            if (parent.pushes.empty()) {
                parent.push_depth = child.push_depth;
                parent.pushes     = child.pushes;
                return true;
            }

            // Only the deepest pushes matter, shallower ones are dropped and rediscovered on later evaluations
            if (child.push_depth > parent.push_depth) {
                parent.push_depth = child.push_depth;
                parent.pushes     = child.pushes;
                return true;
            }
            if (child.push_depth < parent.push_depth) {
                return true;
            }

            // Equal depth, intersect within groups and union across groups
            std::set<std::shared_ptr<Provider>> merged;
            for (const auto& p : parent.pushes) {
                // Providers whose group the child doesn't care about pass straight through
                auto same_group = [&](const auto& c) { return c->type == p->type; };
                if (std::none_of(child.pushes.begin(), child.pushes.end(), same_group)) {
                    merged.insert(p);
                }
                // Otherwise only keep providers both sides can use
                else if (child.pushes.contains(p)) {
                    merged.insert(p);
                }
            }
            for (const auto& p : child.pushes) {
                auto same_group = [&](const auto& c) { return c->type == p->type; };
                if (std::none_of(parent.pushes.begin(), parent.pushes.end(), same_group)) {
                    merged.insert(p);
                }
            }

            // If two requirements needed the same group pushed but had no provider in common we can't push at all
            for (const auto& p : parent.pushes) {
                auto same_group = [&](const auto& c) { return c->type == p->type; };
                if (child.pushes.contains(p) || std::any_of(child.pushes.begin(), child.pushes.end(), same_group)) {
                    if (std::none_of(merged.begin(), merged.end(), same_group)) {
                        return false;
                    }
                }
            }

        parent.pushes = std::move(merged);
        return true;
    }

    Director::Evaluation Director::evaluate_task(const std::shared_ptr<DirectorTask>& task,
                                                 const std::set<std::type_index>& used) {
        return evaluate_group(task->type, task, {}, used);
    }

    Director::Evaluation Director::evaluate_group(const std::type_index& type,
                                                  const std::shared_ptr<DirectorTask>& authority,
                                                  const std::set<std::type_index>& visited,
                                                  const std::set<std::type_index>& used) {

        // A group that doesn't exist can never run and there is nothing to watch that would change that
        if (!groups.contains(type)) {
            return Evaluation();
        }
        auto& group = groups.at(type);

        // Gather the providers we are allowed to consider.
        // If the group is being pushed and we can't beat the pusher's priority we are limited to the pushed provider,
        // otherwise any enabled Provide provider in declaration order.
        std::vector<std::shared_ptr<Provider>> options;
        if (group.pushing_task != nullptr && !challenge_priority(group.pushing_task, authority)) {
            options.push_back(group.pushed_provider);
        }
        else {
            for (const auto& p : group.providers) {
                if (p->classification == Provider::Classification::PROVIDE && p->reaction->enabled) {
                    options.push_back(p);
                }
            }
        }

        // Try each provider in order and take the first that can run, or failing that the first that can push.
        // The watching groups of every provider we rejected along the way are kept so that if any of them change we
        // can reevaluate, the blocked providers may be earlier in the list and therefore preferred.
        Evaluation best;
        for (const auto& p : options) {
            Evaluation e = evaluate_provider(p, authority, visited, used);

            if (e.state == Evaluation::RUNNABLE) {
                e.watching.insert(best.watching.begin(), best.watching.end());
                return e;
            }
            if (e.state == Evaluation::PUSHABLE && best.state != Evaluation::PUSHABLE) {
                // Keep the first pushable option but keep looking for a runnable one
                e.watching.insert(best.watching.begin(), best.watching.end());
                best = std::move(e);
            }
            else {
                best.watching.insert(e.watching.begin(), e.watching.end());
            }
        }

        return best;
    }

    Director::Evaluation Director::evaluate_provider(const std::shared_ptr<Provider>& provider,
                                                     const std::shared_ptr<DirectorTask>& authority,
                                                     std::set<std::type_index> visited,
                                                     std::set<std::type_index> used) {

        Evaluation e;
        e.provider = provider;
        e.watching = {provider->type};

        // This prevents us going in a loop, if we have already looked at this group on this path stop
        if (!visited.insert(provider->type).second) {
            return e;
        }

        // We need to have priority over the currently running task
        if (!challenge_priority(provider->group.active_task, authority)) {
            return e;
        }

        // If this type was previously used by another branch we can't reuse it
        // This can happen when you try to run two sibling tasks that have mutual needs
        if (!used.insert(provider->type).second) {
            return e;
        }

        // Assume we can run until one of our requirements says otherwise.
        // We evaluate every requirement even after one blocks so that we accumulate all the watching groups.
        e.state = Evaluation::RUNNABLE;
        e.used  = std::move(used);

        // Each unmet when condition needs a causing provider pushed before we could run
        for (const auto& w : provider->when) {
            if (!w->current) {
                Evaluation c = evaluate_causing(*w, authority, visited);

                // Whether or not it can be pushed, we are blocked on our own group's state so watch ourselves.
                // The when condition's own state monitor will reevaluate us when the state changes.
                e.watching.insert(provider->type);

                if (c.state == Evaluation::PUSHABLE) {
                    e.state = e.state == Evaluation::BLOCKED ? Evaluation::BLOCKED : Evaluation::PUSHABLE;
                    if (!merge_pushes(e, c)) {
                        e.state = Evaluation::BLOCKED;
                    }
                }
                else {
                    e.state = Evaluation::BLOCKED;
                }
            }
        }

        // Each needs relationship requires its group to be available to us
        for (const auto& n : provider->needs) {
            Evaluation r = evaluate_group(n, authority, visited, e.used);

            e.watching.insert(r.watching.begin(), r.watching.end());

            switch (r.state) {
                case Evaluation::RUNNABLE: {
                    // Take on the requirement's providers and used groups as our own
                    e.providers.push_back(r.provider);
                    e.providers.insert(e.providers.end(), r.providers.begin(), r.providers.end());
                    e.used.insert(r.used.begin(), r.used.end());
                } break;
                case Evaluation::PUSHABLE: {
                    e.state = e.state == Evaluation::BLOCKED ? Evaluation::BLOCKED : Evaluation::PUSHABLE;
                    if (!merge_pushes(e, r)) {
                        e.state = Evaluation::BLOCKED;
                    }
                } break;
                case Evaluation::BLOCKED: {
                    e.state = Evaluation::BLOCKED;
                } break;
            }
        }

        // Only a runnable provider occupies its groups, otherwise we just report why we can't run
        if (e.state != Evaluation::RUNNABLE) {
            e.providers.clear();
            e.used.clear();
        }
        if (e.state == Evaluation::BLOCKED) {
            e.pushes.clear();
            e.push_depth = 0;
        }

        return e;
    }

    Director::Evaluation Director::evaluate_causing(const component::Provider::WhenCondition& when,
                                                    const std::shared_ptr<DirectorTask>& authority,
                                                    const std::set<std::type_index>& visited) {
        Evaluation e;

        // Look through every running group for Provide providers whose Causing statement satisfies this when.
        // A group already needs to be running to push it: pushing redirects a group to a different provider, it
        // never starts a group that nobody asked for.
        for (auto& group_item : groups) {
            auto& group = group_item.second;
            if (group.active_task == nullptr) {
                continue;
            }

            for (const auto& p : group.providers) {
                if (p->classification != Provider::Classification::PROVIDE || !p->causing.contains(when.type)
                    || !when.validator(p->causing.at(when.type))) {
                    continue;
                }

                // If we can't beat the priority of whoever is already pushing this group we can't push it
                if (!challenge_priority(group.pushing_task, authority)) {
                    continue;
                }

                // Evaluate the candidate using the running task's own authority, since if it is pushed it is the
                // group's own task that will be running on it, not ours
                Evaluation c = evaluate_provider(p, group.active_task, visited, {});

                // A candidate that could run right now is a direct push.
                // A candidate that is itself pushable needs its own pushes done first, so it contributes those at a
                // deeper level and will itself be discovered as the next push once they have taken effect.
                Evaluation candidate;
                switch (c.state) {
                    case Evaluation::RUNNABLE: {
                        candidate.state      = Evaluation::PUSHABLE;
                        candidate.push_depth = 0;
                        candidate.pushes     = {p};
                    } break;
                    case Evaluation::PUSHABLE: {
                        candidate.state      = Evaluation::PUSHABLE;
                        candidate.push_depth = c.push_depth + 1;
                        candidate.pushes     = c.pushes;
                    } break;
                    case Evaluation::BLOCKED: continue;
                }

                // Candidates are alternatives, so unlike requirements we prefer the shallowest option, the one
                // fewest pushes away from running. Equal depth candidates are combined.
                if (e.state != Evaluation::PUSHABLE || candidate.push_depth < e.push_depth) {
                    e.state      = Evaluation::PUSHABLE;
                    e.push_depth = candidate.push_depth;
                    e.pushes     = candidate.pushes;
                }
                else if (candidate.push_depth == e.push_depth) {
                    e.pushes.insert(candidate.pushes.begin(), candidate.pushes.end());
                }
            }
        }

        return e;
    }

}  // namespace module::extension
