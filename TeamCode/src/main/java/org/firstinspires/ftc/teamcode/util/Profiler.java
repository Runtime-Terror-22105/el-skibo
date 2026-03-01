package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public final class Profiler {
    private static Profiler INSTANCE;

    public static void init() {
        INSTANCE = new Profiler();
    }

    private static class ProfilePoint {
        private final String _name;
        private long _start;
        private long _duration;
        private final List<ProfilePoint> _children;

        ProfilePoint(String name) {
            this._name = name;
            this._start = -1;
            this._duration = 0;
            this._children = new ArrayList<>();
        }

        void start() {
            this._start = System.nanoTime();
        }

        void end() {
            if (this._start == -1)
                throw new IllegalStateException("ProfilePoint " + _name + " was not started.");
            this._duration += System.nanoTime() - _start;
            this._start = -1;
        }

        String name() {
            return _name;
        }

        long duration() {
            return _duration;
        }

        List<ProfilePoint> children() {
            return _children;
        }

        long selfDuration() {
            long childTime = 0;
            for (ProfilePoint child : children()) {
                childTime += child.duration();
            }
            return duration() - childTime;
        }
    }

    public static final class Scope implements AutoCloseable {
        private Scope(String name) {
            Profiler.push(name);
        }

        @Override
        public void close() {
            Profiler.pop();
        }
    }

    private boolean isStarted = false;
    private ProfilePoint root;
    private List<ProfilePoint> stack = new ArrayList<>();
    private List<ProfilePoint> allNodes = new ArrayList<>();

    private Profiler() {}

    public void startInstance() {
        root = new ProfilePoint("root");
        stack.clear();
        stack.add(root);
        allNodes.clear();
        allNodes.add(root);
        root.start();
        isStarted = true;
    }

    public void endInstance() {
        isStarted = false;
        if (stack.size() != 1)
            throw new IllegalStateException("Profiler stack is not empty at endInstance.");
        root.end();
    }

    private ProfilePoint top() {
        return stack.get(stack.size() - 1);
    }

    public void pushInstance(String name) {
        if (!isStarted) return;

        // Try to find if a child with the same name already exists
        ProfilePoint point = null;
        for (ProfilePoint child : top().children()) {
            if (child.name().equals(name)) {
                point = child;
                break;
            }
        }

        if (point == null) {
            point = new ProfilePoint(name);
            top().children().add(point);
            allNodes.add(point);
        }

        stack.add(point);
        point.start();
    }

    public void popInstance() {
        if (!isStarted) return;

        if (stack.size() <= 1)
            throw new IllegalStateException("Cannot pop root profiler node.");
        ProfilePoint point = stack.remove(stack.size() - 1);
        point.end();
    }

    public List<String> breakdownInstance(int n) {
        List<String> result = new ArrayList<>();
        allNodes.sort((a, b) -> Long.compare(b.selfDuration(), a.selfDuration()));
        for (int i = 0; i < Math.min(n, allNodes.size()); i++) {
            ProfilePoint point = allNodes.get(i);
            double ms = point.selfDuration() / 1_000_000.0;
            result.add(String.format("%s: %.3f ms", point.name(), ms));
        }
        return result;
    }

    private void buildFlamegraph(ProfilePoint point, String prefix, List<String> result) {
        String line = prefix + point.name() + " " + (point.duration() / 1_000_000.0);
        result.add(line);
        for (ProfilePoint child : point.children()) {
            buildFlamegraph(child, prefix + "__", result);
        }
    }

    public List<String> flamegraphInstance() {
        List<String> result = new ArrayList<>();
        buildFlamegraph(root, "", result);
        return result;
    }

    public static void start() {
        if (INSTANCE == null) return;
        INSTANCE.startInstance();
    }

    public static void end() {
        if (INSTANCE == null) return;
        INSTANCE.endInstance();
    }

    public static void push(String name) {
        if (INSTANCE == null) return;
        INSTANCE.pushInstance(name);
    }

    public static void pop() {
        if (INSTANCE == null) return;
        INSTANCE.popInstance();
    }

    public static Scope enter(String name) {
        return new Scope(name);
    }

    public static List<String> breakdown(int n) {
        if (INSTANCE == null) return new ArrayList<>();
        return INSTANCE.breakdownInstance(n);
    }

    public static void sendBreakdown(Telemetry telemetry, int n) {
        if (INSTANCE == null) return;
        List<String> graph = INSTANCE.breakdownInstance(n);
        for (String line : graph) {
            telemetry.addLine(line);
        }
    }

    public static List<String> flamegraph() {
        if (INSTANCE == null) return new ArrayList<>();
        return INSTANCE.flamegraphInstance();
    }

    public static void sendFlamegraph(Telemetry telemetry) {
        if (INSTANCE == null) return;
        List<String> graph = INSTANCE.flamegraphInstance();
        for (String line : graph) {
            telemetry.addLine(line);
        }
    }
}
